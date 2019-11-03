/*
static

      |
     F0  --  F1  --   F2 -- Slow
      |       |        |     /
      |       |          B2
      |       |        /
      |          B1
      |        /
         B0
         |
         CS

dynamic, Fi,j,k means a fast thread and Si means a slow thread.

       F0
      /   \
     Fi   F1
     |   /   \
     |  Fj   F2
     |  |   /   \
     |  |  Fk   Si
     |  |  \    /
     |  |    B2
     |   \  /
     |    B1
     |    /
      \  /
       B0
        |
       CS
        |
reverse upward path
*/

// Recursive versions of the Triangle algorithm
//
// When the Triangle algorithm runs at full contention, half of the threads go via the fast route and the other half are
// routed along the slow route.  It therefore pays to make the slow route as fast as possible.  Why not use the Triangle
// algorithm for this purpose?  Then, within this embedded triangle, we could use the triangle again.  Let us allow a
// nesting of K > 0 levels.  We then need K versions of LamportFast with its shared variables x and y and array b, and K
// versions of Binary.  The calls are entryFast(i, p), exitFast(i, p), and entryBinary(i, b) and exitBinary(i, b), where
// p ranges over the thread numbers, and 0 <= i < K, and b over the booleans (bits).
// 
// There are two versions.  In both versions, we need only modify Figure 2 of the paper (apart from the K systems of
// shared variables of Fast and Binary).  The first version uses an arbitrary slow algorithm as in the paper.  If K = 1,
// this should be just the Triangle algorithm.

#if 0
int function entryTriangle(p) {
	int i, fa = 0;
	while (fa < K && ! entryFast(fa, p)) fa++;
	if (fa == K) entrySlow(p);
	for (i = min(K-1, fa); i >= 0 ; i--) {
		entryBinary(i, i < fa);
	}
	return fa;
}

function exitTriangle(fa, p) {
	int i;
	for (i = 0; i <= min(K-1, fa); i++) {
		exitBinary(i, i < fa);
	}
	if (fa < K) exitFast(fa, p); else exitSlow(p);
}

// ==================================================================

// A biased MX algorithm that gives priority to the lower threads.  I have replaced the tests "i < p" by "i == p".  This
// swaps the two ports of each binary algorithm.

int function entryBiased(p: [0..K]) {
	int i;
	for (i = min(K-1, p); i >= 0 ; i--) {
		entryBinary(i, i == p);
	}
}

function exitBiased(p) {
	int i;
	for (i = 0; i <= min(K-1, p); i++) {
		exitBinary(i, i == p);
	}
}
#endif // 0

// Indeed, independently of the waiting condition, sharing array b would allow starvation of the trapezium.  Scenario:
// use K > 2, and three threads p0, p1, p2.  Thread p0 cycles successfully through LF0.  Threads p1 and p2 are pushed by
// p0 to LF1.  Thread p1 becomes x[1] and thread p2 becomes y[1] and enters the waiting loop as y[1].  Thread p1 exits.
// Thread p2 remains in its waiting loop indefinitely, because each time it inspects b[p0], b[p0] happens to be true.



#include <stdbool.h>

#define inv( c ) ( (c) ^ 1 )

#include "Binary.c"

#ifdef TB

static volatile TYPE **intents CALIGN;					// triangular matrix of intents
static volatile TYPE **turns CALIGN;					// triangular matrix of turns
static unsigned int depth CALIGN;

#else

typedef struct CALIGN {
	Token *ns;											// pointer to path node from leaf to root
	TYPE es;											// left/right opponent
} Tuple;

static Tuple ** states CALIGN;							// handle N threads
static int * levels CALIGN;								// minimal level for binary tree
//static Tuple states[64][6] CALIGN;						// handle 64 threads with maximal tree depth of 6 nodes (lg 64)
//static int levels[64] = { -1 } CALIGN;					// minimal level for binary tree
static Token * t CALIGN;

#endif // TB

//======================================================

static inline void entrySlow(
#ifdef TB
	TYPE id
#else
	int level, Tuple *state
#endif // TB
	) {
#ifdef TB
	unsigned int ridt, ridi;

//	ridi = id;
	for ( unsigned int lv = 0; lv < depth; lv += 1 ) {	// entry protocol
		ridi = id >> lv;								// round id for intent
		ridt = ridi >> 1;								// round id for turn
		intents[lv][ridi] = 1;							// declare intent
		turns[lv][ridt] = ridi;							// RACE
		Fence();										// force store before more loads
		while ( intents[lv][ridi ^ 1] == 1 && turns[lv][ridt] == ridi ) Pause();
//		ridi = ridi >> 1;
	} // for
#else
	for ( int s = 0; s <= level; s += 1 ) {				// entry protocol
		binary_prologue( state[s].es, state[s].ns );
	} // for
#endif // TB
} // entrySlow

static inline void exitSlow(
#ifdef TB
	TYPE id
#else
	int level, Tuple * state
#endif // TB
	) {
#ifdef TB
	for ( int lv = depth - 1; lv >= 0; lv -= 1 ) {		// exit protocol
		intents[lv][id >> lv] = 0;						// retract all intents in reverse order
	} // for
#else
	for ( int s = level; s >= 0; s -= 1 ) {				// exit protocol, reverse order
		binary_epilogue( state[s].es, state[s].ns );
	} // for
#endif // TB
} // exitSlow

//======================================================

enum { K = KKK };
typedef struct CALIGN {
	volatile TYPE * b CALIGN;
	volatile TYPE x CALIGN, y CALIGN;
	volatile Token B; // = { { 0, 0 }, 0 };
} FastPaths;
static FastPaths fastpaths[K] CALIGN;
static TYPE PAD CALIGN __attribute__(( unused ));		// protect further false sharing

#define await( E ) while ( ! (E) ) Pause()


static void * Worker( void * arg ) {
	TYPE id = (size_t)arg;
	uint64_t entry;

#ifdef FAST
	unsigned int cnt = 0, oid = id;
#endif // FAST

#ifndef TB
	int level = levels[id];
	Tuple * state = states[id];
#endif // ! TB

	for ( int r = 0; r < RUNS; r += 1 ) {
		entry = 0;

#ifdef CNT
		for ( unsigned int i = 0; i < CNT + 1; i += 1 ) { // reset for each run
			counters[r][id].cnts[i] = 0;
		} // for
#endif // CNT

		while ( stop == 0 ) {
			intptr_t fa;

#if 1
			for ( fa = 0; fa < K; fa += 1 ) {
				FastPaths * fp = &fastpaths[fa];		// optimization
				if ( FASTPATH( fp->y == N ) ) {
					fp->b[id] = true;
					fp->x = id;
					Fence();							// force store before more loads
					if ( FASTPATH( fp->y == N ) ) {
						fp->y = id;
						Fence();						// force store before more loads
						if ( FASTPATH( fp->x == id ) ) {
							goto Fast;
						} else {
							fp->b[id] = false;
							Fence();					// OPTIONAL, force store before more loads
							for ( uintptr_t k = 0; fp->y == id && k < N; k += 1 )
								await( fp->y != id || ! fp->b[k] );
							if ( FASTPATH( fp->y == id ) ) {
								goto Fast;
							}
						} // if
					} else {
						fp->b[id] = false;
					} // if
				} // if
			} // for
			goto Slow;

#else

			for ( fa = 0; fa < K; fa += 1 ) {
				FastPaths * fp = &fastpaths[fa];		// optimization
				if ( FASTPATH( fp->y != N ) ) continue;
				fp->b[id] = true;						// entry protocol
				fp->x = id;
				Fence();								// force store before more loads
				if ( FASTPATH( fp->y != N ) ) {
					fp->b[id] = false;
					continue;
				} // if
				fp->y = id;
				Fence();								// force store before more loads
				if ( FASTPATH( fp->x != id ) ) {
					fp->b[id] = false;
					Fence();							// OPTIONAL, force store before more loads
					for ( uintptr_t k = 0; fp->y == id && k < N; k += 1 )
						await( fp->y != id || ! fp->b[k] );
					if ( FASTPATH( fp->y != id ) ) continue;
				} // if
				goto Fast;
			} // for
			goto Slow;
#endif // 0

		  Fast: ;
#ifdef CNT
			counters[r][id].cnts[fa] += 1;
#endif // CNT
			for ( intptr_t i = fa; i >= 0; i -= 1 ) {
				binary_prologue( i < fa, &fastpaths[i].B );
			} // for
			CriticalSection( id );
			for ( unsigned int i = 0; i <= fa; i += 1 ) {
				binary_epilogue( i < fa, &fastpaths[i].B );
			} // for

			fastpaths[fa].y = N;						// exit fast protocol
			fastpaths[fa].b[id] = false;
			goto Fini;

		  Slow:
#if defined( __sparc )
			__asm__ __volatile__ ( "" : : : "memory" );
#endif // __sparc

			entrySlow(
#ifdef TB
				id
#else
				level, state
#endif // TB
				);

#ifdef CNT
			counters[r][id].cnts[fa] += 1;
#endif // CNT
			fa -= 1;
			for ( intptr_t i = fa; i >= 0; i -= 1 ) {
				binary_prologue( 1, &fastpaths[i].B );
			} // for
			CriticalSection( id );
			for ( unsigned int i = 0; i <= fa; i += 1 ) {
				binary_epilogue( 1, &fastpaths[i].B );
			} // for

			exitSlow(
#ifdef TB
				id
#else
				level, state
#endif // TB
				);

		  Fini: ;

#ifdef FAST
			id = startpoint( cnt );						// different starting point each experiment
			cnt = cycleUp( cnt, NoStartPoints );
#endif // FAST
			entry += 1;
		} // while

#ifdef FAST
		id = oid;
#endif // FAST
		entries[r][id] = entry;
		__sync_fetch_and_add( &Arrived, 1 );
		while ( stop != 0 ) Pause();
		__sync_fetch_and_add( &Arrived, -1 );
	} // for
	return NULL;
} // Worker

void __attribute__((noinline)) ctor2() {
#ifdef TB
	depth = Clog2( N );									// maximal depth of binary tree
	int width = 1 << depth;								// maximal width of binary tree
	intents = Allocator( sizeof(typeof(intents[0])) * depth ); // allocate matrix columns
	turns = Allocator( sizeof(typeof(turns[0])) * depth );
	for ( unsigned int r = 0; r < depth; r += 1 ) {		// allocate matrix rows
		unsigned int size = width >> r;					// maximal row size
		intents[r] = Allocator( sizeof(typeof(intents[0][0])) * size );
		for ( unsigned int c = 0; c < size; c += 1 ) {	// initial all intents to dont-want-in
			intents[r][c] = 0;
		} // for
		turns[r] = Allocator( sizeof(typeof(turns[0][0])) * (size >> 1) ); // half maximal row size
	} // for
#else
	// element 0 not used
	t = Allocator( sizeof(typeof(t[0])) * N );

	// states[id][s].es indicates the left or right contender at a match.
	// states[id][s].ns is the address of the structure that contains the match data.
	// s ranges from 0 to the tree level of a start point (leaf) in a minimal binary tree.
	// levels[id] is level of start point minus 1 so bi-directional tree traversal is uniform.

	states = Allocator( sizeof(typeof(states[0])) * N );
	levels = Allocator( sizeof(typeof(levels[0])) * N );
	levels[0] = -1;										// default for N=1
	for ( TYPE id = 0; id < N; id += 1 ) {
		t[id].Q[0] = t[id].Q[1] = t[id].R = 0;
		unsigned int start = N + id, level = Log2( start );
		states[id] = Allocator( sizeof(typeof(states[0][0])) * level );
		levels[id] = level - 1;
		for ( unsigned int s = 0; start > 1; start >>= 1, s += 1 ) {
			states[id][s].es = start & 1;
			states[id][s].ns = &t[start >> 1];
		} // for
	} // for
#endif // TB
} // ctor2

void __attribute__((noinline)) ctor() {
	for ( unsigned int k = 0; k < K; k += 1 ) {
		fastpaths[k].b = Allocator( sizeof(typeof(fastpaths[0].b[0])) * N );
		for ( uintptr_t i = 0; i < N; i += 1 ) {		// initialize shared data
			fastpaths[k].b[i] = 0;
		} // for
		fastpaths[k].y = N;
	} // for
	ctor2();											// tournament allocation/initialization
} // ctor

void __attribute__((noinline)) dtor2() {
#ifdef TB
	for ( int r = 0; r < depth; r += 1 ) {				// deallocate matrix rows
		free( (void *)turns[r] );
		free( (void *)intents[r] );
	} // for
	free( (void *)turns );								// deallocate matrix columns
	free( (void *)intents );
#else
	free( (void *)levels );
	free( (void *)states );
	free( (void *)t );
#endif // TB
} // dtor2

void __attribute__((noinline)) dtor() {
	dtor2();											// tournament deallocation
	for ( unsigned int k = 0; k < K; k += 1 ) {
		free( (void *)fastpaths[k].b );
	} // for
} // dtor

// Local Variables: //
// tab-width: 4 //
// compile-command: "gcc -Wall -Wextra -std=gnu11 -O3 -DNDEBUG -fno-reorder-functions -DPIN -DAlgorithm=TriangleTrapezium Harness.c -lpthread -lm" //
// End: //

// Joep L. W. Kessels, Arbitration Without Common Modifiable Variables, Acta Informatica, 17(2), 1982, p. 137

static TYPE PAD1 CALIGN __attribute__(( unused ));		// protect further false sharing
static VTYPE Q[2] CALIGN = { 0, 0 }, R[2] CALIGN;
static TYPE PAD2 CALIGN __attribute__(( unused ));		// protect further false sharing

//#define inv( c ) ((c + 1) % 2)
#define plus( a, b ) ((a + b) % 2)

static void * Worker( void * arg ) {
	TYPE id = (size_t)arg;
	uint64_t entry;

	int other = id ^ 1;									// int is better than TYPE

	for ( int r = 0; r < RUNS; r += 1 ) {
		RTYPE randomThreadChecksum = 0;

		for ( entry = 0; stop == 0; entry += 1 ) {
			#ifdef FLICKER
			for ( int i = 0; i < 100; i += 1 ) Q[id] = i % 2; // flicker
			#endif // FLICKER
			Q[id] = 1;									// entry protocol
			Fence();									// force store before more loads
#if 0
			#ifdef FLICKER
			for ( int i = 0; i < 100; i += 1 ) R[id] = i % 2; // flicker
			#endif // FLICKER
			R[id] = plus( R[other], id );
			Fence();									// force store before more loads
			while ( Q[other] == 1 && R[id] == plus( R[other], id ) ) Pause();
#else
			#ifdef FLICKER
			for ( int i = 0; i < 100; i += 1 ) R[id] = i % 2; // flicker
			#endif // FLICKER
			R[id] = R[other] ^ id ;
			Fence();									// force store before more loads
			while ( Q[other] == 1 && R[id] == (R[other] ^ id) ) Pause() ;
#endif

			randomThreadChecksum += CriticalSection( id );

			#ifdef FLICKER
			for ( int i = 0; i < 100; i += 1 ) Q[id] = i % 2; // flicker
			#endif // FLICKER
			Q[id] = 0;									// exit protocol
		} // for

		__sync_fetch_and_add( &sumOfThreadChecksums, randomThreadChecksum );

		entries[r][id] = entry;
		__sync_fetch_and_add( &Arrived, 1 );
		while ( stop != 0 ) Pause();
		__sync_fetch_and_add( &Arrived, -1 );
	} // for
	return NULL;
} // Worker

void __attribute__((noinline)) ctor() {
	if ( N != 2 ) {
		printf( "\nUsage: N=%zd must be 2\n", N );
		exit( EXIT_FAILURE);
	} // if
} // ctor

void __attribute__((noinline)) dtor() {
} // dtor

// Local Variables: //
// tab-width: 4 //
// compile-command: "gcc -Wall -Wextra -std=gnu11 -O3 -DNDEBUG -fno-reorder-functions -DPIN -DAlgorithm=Kessels2 Harness.c -lpthread -lm -D`hostname` -DCFMT -DCNT=0" //
// End: //

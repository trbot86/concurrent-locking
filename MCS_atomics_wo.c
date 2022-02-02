// John M. Mellor-Crummey and Michael L. Scott, Algorithm for Scalable Synchronization on Shared-Memory Multiprocessors,
// ACM Transactions on Computer Systems, 9(1), 1991, Fig. 5, p. 30

// Architectural fences are unnecssary on TSO for MCS and CLH. An atomic instruction is a hardware fence, hence the only
// reordering is store-load: stores in program order followed by loads being inverted in memory order by the CPU because
// of a store buffer.  (Peter Sewell et al showed all TSO reorderings, even those not arising from the store buffer, are
// attributable to a simple store buffer model, which is a helpful simplification).  In the lock() path, the store to
// clear the flag in the node happens before the atomic preventing movement and that store cannot be reorder in any
// meaningful way.
//
// The other interesting store releases the lock.  Loads after the CS in program order can by lifted by the CPU up above
// the release store into the CS, but these are benign, as it is always safe to move accesses that are outside and after
// the CS "up" into the CS.  Critically, loads and stores in the CS body itself cannot be reordered past the store that
// releases the lock.  Hence, accesses can only "leak" one direction: from outside the CS into the CS, but not the other
// direction. That covers acquire-release memory ordering.
//
// The other concern is compiler-based reordering, but the store to the release the lock should be to a volatile/atomic,
// which protects against compile-time movement.

#include <stdbool.h>

typedef struct mcs_node {
	_Atomic(struct mcs_node *) next;
	_Atomic(int) spin;
} MCS_node CALIGN;

typedef _Atomic(MCS_node *) MCS_lock;

inline void mcs_lock( MCS_lock * lock, MCS_node * node ) {
	__atomic_store_n(&node->next, NULL, __ATOMIC_RELAXED);
	__atomic_store_n(&node->spin, true, __ATOMIC_RELAXED);

	MCS_node * prev = __atomic_exchange_n(lock, node, __ATOMIC_SEQ_CST);

	// no ordering needed here because of data dependence (unless on alpha dec... :-))
    if ( SLOWPATH( prev == NULL ) ) return;						// no one on list ?
	__atomic_store_n(&prev->next, node, __ATOMIC_RELAXED);
	// StLd compiler-only fence needed for progress? (no convenient fence for this exists in C++ spec... normally i'd use an empty volatile asm :::"memory" block...)

	while ( __atomic_load_n(&node->spin, __ATOMIC_RELAXED) == true ) Pause(); // busy wait on my spin variable
	__atomic_thread_fence(__ATOMIC_ACQUIRE); 					// prevent cs RW from being reordered with R above (spinning R happens before CS RW)

	// reasoning about ordering <(happens-after) with __atomic_exchange_n(..., __ATOMIC_SEQ_CST):
	//		CS 	<(acquire fence) spinning
	//			<(seq_cst xchg) node publication
	//		    <(seq_cst xchg) initialization

	// reasoning about insufficiency of ordering when using __ATOMIC_RELAXED for the xchg instead:
	//		CS  <(acquire fence) spinning
	//			<(control dependence for compiler reordering) return if prev null [but nothing for processor reordering!]
	//			<(data dependence on prev) node publication [can speculation break this?]
	//		    <(release fence) initialization

} // mcs_lock

inline void mcs_unlock( MCS_lock * lock, MCS_node * node ) {
	__atomic_thread_fence(__ATOMIC_RELEASE); 					// prevent cs RW from being reordered with W below (CS RW happen before lock release W)
	if ( FASTPATH( __atomic_load_n(&node->next, __ATOMIC_RELAXED) == NULL ) ) {	// no one waiting ?
		MCS_node * temp = node;									// copy because exchange overwrites expected
		// compiler ordering: cas is after load node->next by control dependence
		// processor ordering: seems to me that either the preceding load of node->next must complete first,
		//     or we're speculating, and speculation can't effectively reorder this cas before the load unless
		//     node->next is actually NULL, in which case that reordering is fine...
		if ( __atomic_compare_exchange_n( lock, &temp, NULL, false, __ATOMIC_RELAXED, __ATOMIC_RELAXED ) ) return; // Fence
		// compiler ordering: this load is after the cas by control dependence
		// processor ordering: similar argument that speculation is fine---a bounded number of these loads can be moved before the cas, which is fine if we don't actually return.
		while ( __atomic_load_n(&node->next, __ATOMIC_RELAXED) == NULL ) Pause(); // busy wait until my node is modified
	} // if
	// compiler ordering: this write is after load node->next by control dependence (could need to return in if-block)
	// processor ordering: similar to speculation argument above
	node->next->spin = false;									// stop their busy wait

	// note: i think some reads of node->next can still technically float up into the CS...
	//       but this is fine, since
	//		 - if we incorrectly read node->next == NULL as a result,
	//         we will fail our CAS (which can't move up) and wait until it's null.
	//		 - and we can't incorrectly read node->next != NULL as a result, since
	//         node->next's change to non-null is monotonic until we start our next CS entry attempt

} // mcs_unlock

static TYPE PAD1 CALIGN __attribute__(( unused ));				// protect further false sharing
static MCS_lock lock CALIGN;
static TYPE PAD2 CALIGN __attribute__(( unused ));				// protect further false sharing

static void * Worker( void * arg ) {
	TYPE id = (size_t)arg;
	uint64_t entry;

	#ifdef FAST
	unsigned int cnt = 0, oid = id;
	#endif // FAST

	MCS_node node;

	for ( int r = 0; r < RUNS; r += 1 ) {
		RTYPE randomThreadChecksum = 0;

		for ( entry = 0; stop == 0; entry += 1 ) {
			mcs_lock( &lock, &node );

			randomThreadChecksum += CriticalSection( id );

			mcs_unlock( &lock, &node );

			#ifdef FAST
			id = startpoint( cnt );								// different starting point each experiment
			cnt = cycleUp( cnt, NoStartPoints );
			#endif // FAST
		} // for

		__sync_fetch_and_add( &sumOfThreadChecksums, randomThreadChecksum );

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

void __attribute__((noinline)) ctor() {
	lock = NULL;
} // ctor

void __attribute__((noinline)) dtor() {
} // dtor

// Local Variables: //
// tab-width: 4 //
// compile-command: "gcc -Wall -Wextra -std=gnu11 -O3 -DNDEBUG -fno-reorder-functions -DPIN -DAlgorithm=MCS_atomics_wo Harness.c -lpthread -lm -D`hostname` -DCFMT -DCNT=0" //
// End: //

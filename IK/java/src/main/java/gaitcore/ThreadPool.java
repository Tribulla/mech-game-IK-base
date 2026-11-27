package gaitcore;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.IntConsumer;

/**
 * Persistent work-stealing daemon worker pool mirroring {@code gc_thread.c}.
 * Workers sleep on a condition variable keyed to a monotonic generation counter so per-frame
 * {@link #parallelFor} dispatches allocate zero tasks or futures and include the caller thread.
 */
final class ThreadPool implements AutoCloseable {

    private final int numThreads;
    private final int numWorkers;
    private final ReentrantLock mu = new ReentrantLock();
    private final Condition workCv = mu.newCondition();
    private final Condition doneCv = mu.newCondition();
    private final AtomicInteger nextIdx = new AtomicInteger(0);
    private final Thread[] workers;

    private boolean shutdown;
    private long generation;
    private int activeWorkers;
    private IntConsumer fn;
    private int count;

    ThreadPool(int numThreads) {
        if (numThreads <= 0) {
            numThreads = GaitCore.hardwareThreads();
        }
        if (numThreads < 1) {
            numThreads = 1;
        }
        if (numThreads > 64) {
            numThreads = 64;
        }
        this.numThreads = numThreads;
        // Caller participates directly in parallelFor, so spawn (numThreads - 1) workers.
        this.numWorkers = numThreads - 1;
        if (this.numWorkers > 0) {
            this.workers = new Thread[this.numWorkers];
            for (int i = 0; i < this.numWorkers; i++) {
                Thread t = new Thread(this::workerMain, "gaitcore-worker-" + i);
                t.setDaemon(true);
                t.start();
                this.workers[i] = t;
            }
        } else {
            this.workers = new Thread[0];
        }
    }

    int threadCount() {
        return numThreads;
    }

    private void workerMain() {
        long myGen = 0;
        for (;;) {
            IntConsumer task;
            int itemCount;
            mu.lock();
            try {
                while (!shutdown && generation == myGen) {
                    workCv.awaitUninterruptibly();
                }
                if (shutdown) {
                    return;
                }
                myGen = generation;
                task = fn;
                itemCount = count;
            } finally {
                mu.unlock();
            }

            for (;;) {
                int idx = nextIdx.getAndIncrement();
                if (idx >= itemCount) {
                    break;
                }
                task.accept(idx);
            }

            mu.lock();
            try {
                activeWorkers--;
                if (activeWorkers == 0) {
                    doneCv.signalAll();
                }
            } finally {
                mu.unlock();
            }
        }
    }

    void parallelFor(int itemCount, IntConsumer task) {
        if (task == null || itemCount <= 0) {
            return;
        }
        if (numWorkers <= 0 || itemCount <= 1) {
            for (int i = 0; i < itemCount; i++) {
                task.accept(i);
            }
            return;
        }
        mu.lock();
        try {
            fn = task;
            count = itemCount;
            nextIdx.set(0);
            activeWorkers = numWorkers;
            generation++;
            workCv.signalAll();
        } finally {
            mu.unlock();
        }

        // Caller thread steals work alongside background threads to eliminate idle stalls.
        for (;;) {
            int idx = nextIdx.getAndIncrement();
            if (idx >= itemCount) {
                break;
            }
            task.accept(idx);
        }

        mu.lock();
        try {
            while (activeWorkers > 0) {
                doneCv.awaitUninterruptibly();
            }
        } finally {
            mu.unlock();
        }
    }

    @Override
    public void close() {
        if (numWorkers <= 0) {
            return;
        }
        mu.lock();
        try {
            shutdown = true;
            generation++;
            workCv.signalAll();
        } finally {
            mu.unlock();
        }
        boolean interrupted = false;
        for (Thread w : workers) {
            while (true) {
                try {
                    w.join();
                    break;
                } catch (InterruptedException e) {
                    interrupted = true;
                }
            }
        }
        if (interrupted) {
            Thread.currentThread().interrupt();
        }
    }
}

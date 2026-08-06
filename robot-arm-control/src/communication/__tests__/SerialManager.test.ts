import { SerialManager } from '../SerialManager';

describe('SerialManager command transactions', () => {
  const createManager = (): SerialManager => {
    const manager = new SerialManager();
    (manager as any).writer = {
      write: jest.fn(async () => {}),
      releaseLock: jest.fn()
    };
    return manager;
  };

  test('sendAndAwait resolves on expected OK message', async () => {
    const manager = createManager();
    const promise = manager.sendAndAwait('TEST CMD', {
      expectedTypes: ['OK'],
      timeoutMs: 250,
      retries: 0
    });
    setTimeout(() => {
      (manager as any).processLine('OK test');
    }, 10);

    const response = await promise;
    expect(response.type).toBe('OK');
    expect((manager as any).writer.write).toHaveBeenCalledTimes(1);
  });

  test('clearTrajectoryQueue waits for TQ_READY acknowledgement', async () => {
    const manager = createManager();
    const promise = manager.clearTrajectoryQueue();
    setTimeout(() => {
      (manager as any).processLine('TQ READY 0');
    }, 10);

    await promise;
    expect((manager as any).writer.write).toHaveBeenCalledTimes(1);
  });

  test('sendAndAwait rejects when firmware returns ERROR', async () => {
    const manager = createManager();
    const promise = manager.sendAndAwait('TEST FAIL', {
      expectedTypes: ['OK'],
      timeoutMs: 250,
      retries: 0
    });
    setTimeout(() => {
      (manager as any).processLine('ERROR simulated failure');
    }, 10);

    await expect(promise).rejects.toThrow('simulated failure');
  });

  test('sendAndAwait surfaces typed queue error detail', async () => {
    const manager = createManager();
    const promise = manager.sendAndAwait('TQ PT 0 0 0 0 0 0 0 0 0 0 0 0 0', {
      expectedTypes: ['OK'],
      timeoutMs: 250,
      retries: 0
    });
    setTimeout(() => {
      (manager as any).processLine('TQ ERR TIME_NON_MONOTONIC time not monotonic');
    }, 10);

    await expect(promise).rejects.toThrow('TQ TIME_NON_MONOTONIC: time not monotonic');
  });

  test('sendAndAwait tags generic device ERROR with queue command scope', async () => {
    const manager = createManager();
    const promise = manager.sendAndAwait('TQ RUN', {
      expectedTypes: ['OK'],
      timeoutMs: 250,
      retries: 0
    });
    setTimeout(() => {
      (manager as any).processLine('ERROR');
    }, 10);

    await expect(promise).rejects.toThrow('TQ DEVICE_ERROR');
  });
});

export const createCartesianPlannerWorker = (): Worker => (
  new Worker(new URL('./cartesianPlannerWorker.ts', import.meta.url))
);

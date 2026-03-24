describe('FEATURE_IK_COLLISION_CHECK_V1 flag evaluation', () => {
  const evaluate = (envVal: string | undefined): boolean =>
    envVal !== '0';

  it('is true when env var is absent', () => {
    expect(evaluate(undefined)).toBe(true);
  });

  it('is true when env var is set to 1', () => {
    expect(evaluate('1')).toBe(true);
  });

  it('is false when env var is set to 0 (opt-out)', () => {
    expect(evaluate('0')).toBe(false);
  });
});

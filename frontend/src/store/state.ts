export const defaultState = {
  cycleStarted: false,
  resistance: 0,
  robotConsumption: {
    batteryPercentage: 0,
    batteryRemainingTimeInSeconds: 0,
  },
};

export const state = {
  cycleStarted: defaultState.cycleStarted,
  resistance: defaultState.resistance,
  robotConsumption: defaultState.robotConsumption,
  // TODO : Implement other state values
};

export type State = typeof state;

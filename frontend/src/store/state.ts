export const state = {
  socket: {
    isConnected: false,
    reconnectError: false,
  },
};

export type State = typeof state;

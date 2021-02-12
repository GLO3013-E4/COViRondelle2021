import { enumFactory } from 'node-factory';

export const COLORS = ['yellow', 'brown', 'red', 'pink', 'orange','black', 'white', 'green', 'blue', 'purple','grey'];

export const StateFactory = enumFactory<string>(COLORS);
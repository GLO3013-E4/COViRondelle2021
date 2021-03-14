import { factory } from 'node-factory';
import { defaultState, State } from '@/store/state';
import { ColorFactory } from '@/factories/ColorFactory';
import { CornerFactory } from '@/factories/CornerFactory';
import { CoordinateFactory } from '@/factories/CoordinateFactory';

export const StateFactory = factory<State>((fake) => ({
  // TODO : Fake what isn't faked when implementing
  cycleReady: defaultState.cycleReady,
  cycleStarted: defaultState.cycleStarted,
  tableImage: defaultState.tableImage,
  // TODO : Find a way to implement ResistanceFactory
  resistance: fake.random.number(10000),
  batteryConsumption: defaultState.batteryConsumption,
  robotConsumption: defaultState.robotConsumption,
  puckColors: ColorFactory.get(3),
  puckFirstCorner: CornerFactory.get(),
  plannedTrajectory: CoordinateFactory.make(fake.random.number(20)),
  realTrajectory: CoordinateFactory.make(fake.random.number(20)),
  puckInGrip: fake.random.boolean(),
  currentStep: defaultState.currentStep,
}));

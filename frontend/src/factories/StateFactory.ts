import { factory } from 'node-factory';
import { defaultState, State } from '@/store/state';
import { ColorFactory } from '@/factories/ColorFactory';
import { CornerFactory } from '@/factories/CornerFactory';
import { CoordinateFactory } from '@/factories/CoordinateFactory';
import { Coordinate } from '@/types/coordinate';

const TRAJECTORY_POINTS = 20;
const CURRENT_TRAJECTORY_POINTS = TRAJECTORY_POINTS / 2;

export const StateFactory = factory<State>((fake) => {
  const plannedTrajectory = CoordinateFactory.make(TRAJECTORY_POINTS);

  const fakeRealPoints = (coordinates: Array<Coordinate>) => {
    return coordinates.map((coordinate) => {
      const fakeFactor =
        (fake.random.boolean() ? -1 : 1) * fake.random.number(20);
      return {
        x: coordinate.x + fakeFactor,
        y: coordinate.y + fakeFactor,
      } as Coordinate;
    });
  };

  return {
    // TODO : Fake what isn't faked when implementing
    cycleReady: defaultState.cycleReady,
    cycleStarted: defaultState.cycleStarted,
    tableImage: defaultState.tableImage,
    // TODO : Find a way to implement ResistanceFactory
    resistance: fake.random.number(10000),
    robotConsumption: defaultState.robotConsumption,
    puckColors: ColorFactory.get(3),
    puckFirstCorner: CornerFactory.get(),
    plannedTrajectory,
    currentPlannedTrajectory: plannedTrajectory.splice(
      0,
      CURRENT_TRAJECTORY_POINTS
    ),
    realTrajectory: fakeRealPoints(plannedTrajectory),
    puckInGrip: fake.random.boolean(),
    currentStep: defaultState.currentStep,
  };
});

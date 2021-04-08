import { factory } from 'node-factory';
import { defaultState, State } from '@/store/state';
import { ColorFactory } from '@/factories/ColorFactory';
import { CornerFactory } from '@/factories/CornerFactory';
import { CoordinateFactory } from '@/factories/CoordinateFactory';
import { Coordinate } from '@/types/coordinate';
import { PuckList } from '@/types/puckList';

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

  const fakePuckColors = ColorFactory.get(3);
  const fakePuckList: PuckList = defaultState.puckList;
  fakePuckList.setPuckColors(fakePuckColors);

  return {
    // TODO : Fake what isn't faked when implementing
    cycleReady: defaultState.cycleReady,
    cycleStarted: defaultState.cycleStarted,
    tableImage: '/stub_table_image.jpg',
    // TODO : Find a way to implement ResistanceFactory
    resistance: fake.random.number(10000),
    robotConsumption: defaultState.robotConsumption,
    puckColors: fakePuckColors,
    puckFirstCorner: CornerFactory.get(),
    plannedTrajectory,
    currentPlannedTrajectory: plannedTrajectory.slice(
      0,
      CURRENT_TRAJECTORY_POINTS
    ),
    realTrajectory: fakeRealPoints(plannedTrajectory),
    puckInGrip: fake.random.boolean(),
    currentStep: defaultState.currentStep,
    puckList: fakePuckList,
  };
});

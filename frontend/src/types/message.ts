import { RobotConsumption } from '@/types/robotConsumption';
import { Color } from '@/types/color';
import { Corner } from '@/types/corner';
import { Coordinate } from '@/types/coordinate';
import { GripState } from '@/types/gripState';
import { Step } from '@/types/step';

export interface Message {
  resistance?: number;
  robotConsumption?: RobotConsumption;
  puckColors?: Array<Color>;
  puckFirstCorner?: Corner;
  plannedTrajectoryCoordinate?: Coordinate;
  realTrajectoryCoordinate?: Coordinate;
  gripState?: GripState;
  currentStep?: Step;
}

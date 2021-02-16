import { RobotConsumption } from '@/types/robotConsumption';
import { Color } from '@/types/color';
import { Corner } from '@/types/corner';
import { Coordinate } from '@/types/coordinate';
import { GripState } from '@/types/gripState';
import { Step } from '@/types/step';

export interface Message {
  resistance?: number;
  tableImage?: string; // TODO : Table image most likely won't be a string, this is temporary
  robotConsumption?: RobotConsumption;
  puckColors?: Array<Color>;
  puckFirstCorner?: Corner;
  plannedTrajectoryCoordinate?: Coordinate;
  realTrajectoryCoordinate?: Coordinate;
  gripState?: GripState;
  currentStep?: Step;
}

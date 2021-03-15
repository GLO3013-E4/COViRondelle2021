import { BatteryConsumption } from '@/types/batteryConsumption';
import { Color } from '@/types/color';
import { Corner } from '@/types/corner';
import { Coordinate } from '@/types/coordinate';
import { Step } from '@/types/step';
import { RobotConsumption } from './robotConsumption';

export interface Message {
  resistance?: number;
  tableImage?: string; // TODO : Table image most likely won't be a string, this is temporary
  batteryConsumption?: BatteryConsumption;
  robotConsumption?: RobotConsumption;
  puckColors?: Array<Color>;
  puckFirstCorner?: Corner;
  plannedTrajectoryCoordinates?: Array<Coordinate>;
  realTrajectoryCoordinate?: Coordinate;
  puckInGrip?: boolean;
  currentStep?: Step;
}

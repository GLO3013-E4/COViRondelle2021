import { RobotConsumption } from '@/types/robotConsumption';
import { Color } from '@/types/color';

export interface Message {
  resistance?: number;
  robotConsumption?: RobotConsumption;
  puckColors?: Array<Color>;
}

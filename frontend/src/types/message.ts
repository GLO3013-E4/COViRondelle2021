import { RobotConsumption } from '@/types/robotConsumption';
import { Color } from '@/types/color';
import { Corner } from '@/types/corner';

export interface Message {
  resistance?: number;
  robotConsumption?: RobotConsumption;
  puckColors?: Array<Color>;
  puckFirstCorner?: Corner;
}

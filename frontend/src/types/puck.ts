import { Color } from '@/types/color';
import { PuckState } from '@/types/puckState';
import { Corner } from '@/types/corner';

export class Puck {
  color = Color.White;
  corner = Corner.A;
  state = PuckState.UNTOUCHED;

  isUntouched = (): boolean => this.state == PuckState.UNTOUCHED;

  isGripped = (): boolean => this.state == PuckState.GRIPPED;
}

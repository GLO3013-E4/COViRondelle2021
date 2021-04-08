import { Color } from '@/types/color';
import { PuckState } from '@/types/puckState';

export class Puck {
  color = Color.Transparent;
  state = PuckState.UNTOUCHED;

  isGripped = (): boolean => this.state == PuckState.GRIPPED;
}

import { Color } from '@/types/color';
import { PuckState } from '@/types/puckState';
import { Corner } from '@/types/corner';

export class Puck {
  color = Color.White;
  corner = Corner.A;
  state = PuckState.UNTOUCHED;

  get isUntouched(): boolean {
    return this.state == PuckState.UNTOUCHED;
  }

  get isGripped(): boolean {
    return this.state == PuckState.GRIPPED;
  }

  get isDeposited(): boolean {
    return this.state == PuckState.RELEASED;
  }
}

import { Puck } from '@/types/puck';
import { Color } from '@/types/color';

export class PuckList {
  private readonly PUCKS_COUNT = 3;
  private pucks: Array<Puck> = Array(this.PUCKS_COUNT).fill(new Puck());

  hasOneGripped = (): boolean => this.pucks.some((puck) => puck.isGripped());

  setPuckColors(puckColors: Array<Color>) {
    if (puckColors.length == this.PUCKS_COUNT) {
      this.pucks.forEach((puck, index) => (puck.color = puckColors[index]));
    }
  }
}

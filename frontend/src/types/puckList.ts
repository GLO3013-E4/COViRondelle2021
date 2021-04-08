import {Puck} from '@/types/puck';
import {Color} from '@/types/color';
import {Corner, getNextCorner} from "@/types/corner";
import {PuckState} from "@/types/puckState";

export class PuckList {
  private readonly PUCKS_COUNT = 3;
  private pucks: Array<Puck> = Array(this.PUCKS_COUNT).fill(new Puck());

  get hasOneGripped(): boolean {
    return this.pucks.some((puck) => puck.isGripped());
  }

  set hasOneGripped(hasOneGripped: boolean) {
    if (hasOneGripped) {
      this.setPuckState(Puck.prototype.isUntouched, PuckState.GRIPPED);
    } else {
      this.setPuckState(Puck.prototype.isGripped, PuckState.DEPOSITED);
    }
  }

  private setPuckState(puckFunction: () => boolean, newState: PuckState) {
    const puck = this.pucks.find(puck => puckFunction.call(puck))
    if (puck) {
      puck.state = newState;
    }
  }

  get first(): Puck {
    return this.pucks[0];
  }

  get colors(): Array<Color> {
    return this.pucks.map(puck => puck.color);
  }

  set colors(colors: Array<Color>) {
    if (colors.length == this.PUCKS_COUNT) {
      this.pucks.forEach((puck, index) => (puck.color = colors[index]));
    }
  }

  set firstCorner(corner: Corner) {
    this.first.corner = corner;
    let nextCorner = corner;

    for (let i = 1; i < this.PUCKS_COUNT; i++) {
      nextCorner = getNextCorner(nextCorner);
      this.pucks[i].corner = nextCorner;
    }
  }
}

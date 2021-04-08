import { PuckList } from '@/types/puckList';
import { PuckFactory } from '@/factories/PuckFactory';

export class PuckListFactory {
  static make = () => {
    const pucks = [];
    for (let index = 0; index < PuckList.PUCKS_COUNT; index++) {
      pucks.push(PuckFactory.make());
    }
    return new PuckList(pucks);
  };
}

import { PuckList } from '@/types/puckList';
import { PuckFactory } from '@/factories/PuckFactory';

export class PuckListFactory {
    static make = () => new PuckList(PuckFactory.make(PuckList.PUCKS_COUNT));
}

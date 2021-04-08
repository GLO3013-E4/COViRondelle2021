import { PuckList } from '@/types/puckList';
import { factory } from 'node-factory';
import { PuckFactory } from '@/factories/PuckFactory';

export const PuckListFactory = factory<PuckList>(() => {
  return {
    pucks: PuckFactory.make(PuckList.PUCKS_COUNT),
  } as PuckList;
});

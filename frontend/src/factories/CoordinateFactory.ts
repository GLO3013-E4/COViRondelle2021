import { factory } from 'node-factory';
import { Coordinate } from '@/types/coordinate';

export const CoordinateFactory = factory<Coordinate>((fake) => ({
  x: fake.random.number(2000),
  y: fake.random.number(2000),
}));

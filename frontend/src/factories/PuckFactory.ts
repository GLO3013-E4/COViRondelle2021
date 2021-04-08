import { factory } from 'node-factory';
import { ColorFactory } from '@/factories/ColorFactory';
import { CornerFactory } from '@/factories/CornerFactory';
import { PuckStateFactory } from '@/factories/PuckStateFactory';
import { Puck } from '@/types/puck';

export const PuckFactory = factory<Puck>(() =>
    new Puck(
        ColorFactory.get(),
        CornerFactory.get(),
        PuckStateFactory.get()
    ));

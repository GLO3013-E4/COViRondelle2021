import { enumFactory } from 'node-factory';
import { Color } from '@/types/color';

export const ColorFactory = enumFactory<string>(Object.values(Color));

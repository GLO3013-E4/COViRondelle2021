import { enumFactory } from 'node-factory';
import ControlPanelResult from '../classes/ControlPanelResult';
import { Corner } from '@/types/corner';

export const CONTROL_PANEL_RESULT = [
  new ControlPanelResult(Corner.A),
  new ControlPanelResult(Corner.B),
  new ControlPanelResult(Corner.C),
  new ControlPanelResult(Corner.D),
];

// TODO : Remove this file
export const ControlPanelResultFactory = enumFactory<ControlPanelResult>(
  CONTROL_PANEL_RESULT
);

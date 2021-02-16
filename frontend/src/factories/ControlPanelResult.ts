import { enumFactory } from 'node-factory';
import ControlPanelResult from '../classes/ControlPanelResult';

export const CONTROL_PANEL_RESULT = [
  new ControlPanelResult('A'),
  new ControlPanelResult('B'),
  new ControlPanelResult('C'),
  new ControlPanelResult('D'),
];

export const ControlPanelResultFactory = enumFactory<ControlPanelResult>(
  CONTROL_PANEL_RESULT
);

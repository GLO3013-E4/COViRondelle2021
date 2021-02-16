import { Corner } from '@/types/corner';

export default class ControlPanelResult {
  corner: Corner;
  placementLeft: boolean;
  placementBottom: boolean;

  constructor(corner: Corner) {
    this.corner = corner;

    this.placementLeft = false;
    this.placementBottom = false;
    this.initializePlacement();
  }

  private initializePlacement() {
    switch (this.corner) {
      case Corner.A: {
        this.placementLeft = true;
        this.placementBottom = false;
        break;
      }
      case Corner.B: {
        this.placementLeft = false;
        this.placementBottom = true;
        break;
      }
      case Corner.C: {
        this.placementLeft = true;
        this.placementBottom = true;
        break;
      }
      default: {
        this.placementLeft = false;
        this.placementBottom = false;
        break;
      }
    }
  }
}

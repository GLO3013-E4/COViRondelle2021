export default class ControlPanelResult {
    corner:string | undefined;
    placementLeft:boolean;
    placementBottom:boolean;

    constructor(corner:string){
        this.corner = corner;
        
        this.placementLeft = false;
        this.placementBottom = false;
        this.initializePlacement();
    }

    private initializePlacement() {
        switch(this.corner) { 
            case "A": { 
                this.placementLeft = true;
                this.placementBottom = false;
               break; 
            } 
            case "C": {
                this.placementLeft = false;
                this.placementBottom = true;
               break; 
            }
            case "D": { 
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
export const fr = {
  appName: 'Robot culinaire',
  trajectories: {
    plannedTrajectory: 'Trajectoire planifiée',
    realTrajectory: 'Trajectoire réelle',
  },
  station: {
    stationInformation: 'Information de la station',
    resistance: 'Résistance',
    gripState: 'État du préhenseur',
    puckInGrip: 'Rondelle présente',
    noPuck: 'Pas de rondelle',
    puckDeposit: 'Dépôt de rondelles',
    firstCorner: 'Premier coin',
  },
  cycles: {
    steps: {
      CycleNotStarted: 'Cycle non débuté',
      CycleReadyInWaitingMode: 'Cycle prêt, en attente',
      CycleStarted: 'Cycle débuté',
      ToResistanceStation: 'Vers la station de résistance',
      ReadResistance: 'Lecture de la résistance',
      ToControlPanel: 'Vers le panneau de contrôle',
      ReadControlPanel: 'Lecture du panneau de contrôle',
      ToFirstPuckAndGrabFirstPuck: 'Vers la première rondelle et préhension',
      ToFirstCornerAndReleaseFirstPuck: 'Vers le premier coin et relâchement',
      ToSecondPuckAndGrabSecondPuck: 'Vers la deuxième rondelle et préhension',
      ToSecondCornerAndReleaseSecondPuck:
        'Vers le deuxième coin et relâchement',
      ToThirdPuckAndGrabThirdPuck: 'Vers la troisième rondelle et préhension',
      ToThirdCornerAndReleaseThirdPuck: 'Vers le troisième coin et relâchement',
      ToSquareCenter: 'Vers le centre du carré',
      CycleEndedAndRedLedOn: 'Fin du cycle et activation du LED rouge',
    },
    start: 'Commencer',
    modes: {
      booting: 'Démarrage',
      started: 'Débuté',
      waiting: 'Attente',
      noInformation: "Pas d'information",
    },
  },
};

<template>
  <v-card color="lighten2">
    <v-container>
      <v-row>
        <v-col sm="12">
          <v-card class="d-flex justify-center lighten1">
            <h3>{{ $t('trajectories.legend') }}</h3>
          </v-card>
        </v-col>
      </v-row>
      <v-row no-gutters>
        <svg height="120" width="400" id="legend">
          <text :x="xInitialPlacement" :y="yInitialPlacement" fill="blue">
            {{ `${$t('trajectories.plannedTrajectory')} :` }}
          </text>
          <line
            :x1="x1StrokeInitialPlacement"
            :y1="yStrokeInitialPlacement"
            :x2="x2StrokeInitialPlacement"
            :y2="yStrokeInitialPlacement"
            style="stroke: blue; stroke-width: 2"
          />
          <text
            :x="xInitialPlacement"
            :y="yInitialPlacement + deltaRealTrajectory"
            fill="red"
          >
            {{ `${$t('trajectories.realTrajectory')} :` }}
          </text>
          <line
            :x1="x1StrokeInitialPlacement"
            :y1="yStrokeInitialPlacement + deltaRealTrajectory"
            :x2="x2StrokeInitialPlacement"
            :y2="yStrokeInitialPlacement + deltaRealTrajectory"
            style="stroke: red; stroke-width: 2"
          />
          <text
            :x="xInitialPlacement"
            :y="yInitialPlacement + deltaStartingPoint"
            fill="red"
          >
            {{ `${$t('trajectories.startingPoint')} :` }}
          </text>
          <circle
            :cx="x1StrokeInitialPlacement"
            :cy="yInitialPlacement + deltaStartingPoint - 5"
            r="2"
            stroke="red"
            stroke-width="2"
            fill="none"
          />
          <text
            :x="xInitialPlacement"
            :y="yInitialPlacement + deltaEndingPoint"
            fill="green"
          >
            {{ `${$t('trajectories.destinationPoint')} :` }}
          </text>
          <circle
            :cx="x1StrokeInitialPlacement"
            :cy="yInitialPlacement + deltaEndingPoint - 5"
            r="2"
            stroke="green"
            stroke-width="2"
            fill="none"
          />
        </svg>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';

@Component({})
export default class Legend extends Vue {
  public xInitialPlacement = 10;
  public yInitialPlacement = 20;

  public x1StrokeInitialPlacement = 180;
  public x2StrokeInitialPlacement = 250;
  public yStrokeInitialPlacement = 15;

  public deltaRealTrajectory = 30;
  public deltaStartingPoint = 60;
  public deltaEndingPoint = 90;
}
</script>

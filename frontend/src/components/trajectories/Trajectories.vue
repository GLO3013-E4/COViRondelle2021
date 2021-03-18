<template>
  <v-card
      class="d-flex justify-center mb-10"
      color="#ededed"
      :height="this.rescaleHeight + 200"
      :width="this.rescaleWidth + 50"
  >
    <v-container>
      <v-row>
        <v-col sm="12">
          <v-card class="d-flex justify-center">
            <h3>{{ $t('trajectories.trajectories') }}</h3>
          </v-card>
        </v-col>
      </v-row>
      <v-row no-gutters>
        <v-col
            class="path"
            v-bind:style="{
            background: `url('${tableImage}')`,
            backgroundSize: `${this.rescaleWidth}px ${this.rescaleHeight}px`,
            backgroundRepeat: 'no-repeat',
          }"
        >
          <svg :height="this.rescaleHeight" :width="this.rescaleWidth" id="svg">
            <polyline
                id="planned_path"
                :points="this.plannedTrajectoryPoints"
                style="fill: none; stroke: blue; stroke-width: 2"
            />
            <polyline
                id="real_path"
                :points="this.realTrajectoryPoints"
                style="fill: none; stroke: red; stroke-width: 2"
            />
            <circle
                class="start"
                :cx="this.startPointX"
                :cy="this.startPointY"
                r="2"
                stroke="red"
                stroke-width="2"
                fill="none"
            />
            <circle
                class="destination"
                :cx="this.destinationPointX"
                :cy="this.destinationPointY"
                r="2"
                stroke="green"
                stroke-width="2"
                fill="none"
            />
          </svg>
        </v-col>
        <v-col sm="12">
          <v-card-actions class="d-flex left">
            <svg
                :height="this.rescaleHeight / 2"
                :width="this.rescaleWidth"
                id="legend"
            >
              <text
                  :x="this.rescaleWidth / 2"
                  :y="this.rescaleHeight / 20"
                  fill="blue"
              >
                {{ `${$t('trajectories.plannedTrajectory')} :` }}
              </text>
              <line
                  :x1="this.rescaleWidth - this.rescaleWidth / 6"
                  :y1="this.rescaleHeight / 20 - 5"
                  :x2="this.rescaleWidth - this.rescaleWidth / 12"
                  :y2="this.rescaleHeight / 20 - 5"
                  style="stroke: blue; stroke-width: 2"
              />
              <text
                  :x="this.rescaleWidth / 2"
                  :y="this.rescaleHeight / 8"
                  fill="red"
              >
                {{ `${$t('trajectories.realTrajectory')} :` }}
              </text>
              <line
                  :x1="this.rescaleWidth - this.rescaleWidth / 6"
                  :y1="this.rescaleHeight / 8 - 5"
                  :x2="this.rescaleWidth - this.rescaleWidth / 12"
                  :y2="this.rescaleHeight / 8 - 5"
                  style="stroke: red; stroke-width: 2"
              />
              <text
                  :x="this.rescaleWidth / 2"
                  :y="this.rescaleHeight / 5"
                  fill="red"
              >
                {{ `${$t('trajectories.startingPoint')} :` }}
              </text>
              <circle
                  :cx="this.rescaleWidth - this.rescaleWidth / 6"
                  :cy="this.rescaleHeight / 5 - 5"
                  r="2"
                  stroke="red"
                  stroke-width="2"
                  fill="none"
              />
              <text
                  :x="this.rescaleWidth / 2"
                  :y="(13 * this.rescaleHeight) / 48"
                  fill="green"
              >
                {{ `${$t('trajectories.destinationPoint')} :` }}
              </text>
              <circle
                  :cx="this.rescaleWidth - this.rescaleWidth / 6"
                  :cy="(13 * this.rescaleHeight) / 48 - 5"
                  r="2"
                  stroke="green"
                  stroke-width="2"
                  fill="none"
              />
            </svg>
          </v-card-actions>
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import {Component, Vue} from 'vue-property-decorator';
import {Coordinate} from '@/types/coordinate';
import {mapState} from 'vuex';
import {State} from "@/store/state";

const ratioX = 0.5;
const ratioY = 0.5;

const coordinatesToString = (trajectory: Array<Coordinate>): string => {
  let points = '';
  trajectory.forEach(
      (coordinate) =>
          (points += `${coordinate.x * ratioX},${
              coordinate.y * ratioY
          } `)
  );
  return points;
}

@Component({
  computed: mapState({
    tableImage: (state: State) => state.tableImage,
    plannedTrajectoryPoints: (state: State) => coordinatesToString(state.plannedTrajectory),
    realTrajectoryPoints: (state: State) => coordinatesToString(state.realTrajectory),
    startPointX: (state: State) => state.plannedTrajectory[0]
        ? state.plannedTrajectory[0].x * ratioX
        : 0,
    startPointY: (state: State) => state.plannedTrajectory[0]
        ? state.plannedTrajectory[0].y * ratioY
        : 0,
    destinationPointX: (state: State) => state.plannedTrajectory[state.plannedTrajectory.length - 1]
        ? state.plannedTrajectory[state.plannedTrajectory.length - 1].x *
        ratioX
        : 0,
    destinationPointY: (state: State) => state.plannedTrajectory[state.plannedTrajectory.length - 1]
        ? state.plannedTrajectory[state.plannedTrajectory.length - 1].y *
        ratioY
        : 0,
  })
})
export default class Trajectories extends Vue {
  private tableImage!: string;
  private plannedTrajectoryPoints!: string;
  private realTrajectoryPoints!: string;
  private startPointX!: number;
  private startPointY!: number;
  private destinationPointX!: number;
  private destinationPointY!: number;
  private readonly width = 1600;
  private readonly height = 904;

  private get rescaleWidth() {
    return this.width * ratioX;
  }

  private get rescaleHeight() {
    return this.height * ratioY;
  }
}
</script>

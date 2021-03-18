<template>
  <v-card
    class="d-flex justify-center mb-10"
    color="#ededed"
    :height="this.rescaledHeight + 200"
    :width="this.rescaledWidth + 50"
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
            backgroundSize: `${this.rescaledWidth}px ${this.rescaledHeight}px`,
            backgroundRepeat: 'no-repeat',
          }"
        >
          <svg
            :height="this.rescaledHeight"
            :width="this.rescaledWidth"
            id="svg"
          >
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
              :height="this.rescaledHeight / 2"
              :width="this.rescaledWidth"
              id="legend"
            >
              <text
                :x="this.rescaledWidth / 2"
                :y="this.rescaledHeight / 20"
                fill="blue"
              >
                {{ `${$t('trajectories.plannedTrajectory')} :` }}
              </text>
              <line
                :x1="this.rescaledWidth - this.rescaledWidth / 6"
                :y1="this.rescaledHeight / 20 - 5"
                :x2="this.rescaledWidth - this.rescaledWidth / 12"
                :y2="this.rescaledHeight / 20 - 5"
                style="stroke: blue; stroke-width: 2"
              />
              <text
                :x="this.rescaledWidth / 2"
                :y="this.rescaledHeight / 8"
                fill="red"
              >
                {{ `${$t('trajectories.realTrajectory')} :` }}
              </text>
              <line
                :x1="this.rescaledWidth - this.rescaledWidth / 6"
                :y1="this.rescaledHeight / 8 - 5"
                :x2="this.rescaledWidth - this.rescaledWidth / 12"
                :y2="this.rescaledHeight / 8 - 5"
                style="stroke: red; stroke-width: 2"
              />
              <text
                :x="this.rescaledWidth / 2"
                :y="this.rescaledHeight / 5"
                fill="red"
              >
                {{ `${$t('trajectories.startingPoint')} :` }}
              </text>
              <circle
                :cx="this.rescaledWidth - this.rescaledWidth / 6"
                :cy="this.rescaledHeight / 5 - 5"
                r="2"
                stroke="red"
                stroke-width="2"
                fill="none"
              />
              <text
                :x="this.rescaledWidth / 2"
                :y="(13 * this.rescaledHeight) / 48"
                fill="green"
              >
                {{ `${$t('trajectories.destinationPoint')} :` }}
              </text>
              <circle
                :cx="this.rescaledWidth - this.rescaledWidth / 6"
                :cy="(13 * this.rescaledHeight) / 48 - 5"
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
import { Component, Vue } from 'vue-property-decorator';
import { Coordinate } from '@/types/coordinate';
import { mapState } from 'vuex';
import { State } from '@/store/state';

const width = 1600;
const height = 904;
const ratioX = 0.5;
const ratioY = 0.5;

const coordinatesToString = (trajectory: Array<Coordinate>): string => {
  let points = '';
  trajectory.forEach(
    (coordinate) =>
      (points += `${coordinate.x * ratioX},${coordinate.y * ratioY} `)
  );
  return points;
};

@Component({
  computed: {
    rescaledWidth: () => width * ratioX,
    rescaledHeight: () => height * ratioY,
    ...mapState({
      tableImage: (state: State) => state.tableImage,
      plannedTrajectoryPoints: (state: State) =>
        coordinatesToString(state.plannedTrajectory),
      realTrajectoryPoints: (state: State) =>
        coordinatesToString(state.realTrajectory),
      startPointX: (state: State) =>
        state.plannedTrajectory[0] ? state.plannedTrajectory[0].x * ratioX : 0,
      startPointY: (state: State) =>
        state.plannedTrajectory[0] ? state.plannedTrajectory[0].y * ratioY : 0,
      destinationPointX: (state: State) =>
        state.plannedTrajectory[state.plannedTrajectory.length - 1]
          ? state.plannedTrajectory[state.plannedTrajectory.length - 1].x *
            ratioX
          : 0,
      destinationPointY: (state: State) =>
        state.plannedTrajectory[state.plannedTrajectory.length - 1]
          ? state.plannedTrajectory[state.plannedTrajectory.length - 1].y *
            ratioY
          : 0,
    }),
  },
})
export default class Trajectories extends Vue {
  private rescaledWidth!: number;
  private rescaledHeight!: number;
  private tableImage!: string;
  private plannedTrajectoryPoints!: string;
  private realTrajectoryPoints!: string;
  private startPointX!: number;
  private startPointY!: number;
  private destinationPointX!: number;
  private destinationPointY!: number;
}
</script>

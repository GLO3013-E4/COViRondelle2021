<template>
  <v-card class="d-flex justify-center mb-10" color="base">
    <v-container>
      <v-row>
        <v-col sm="12">
          <v-card class="d-flex justify-center">
            <h3>{{ $t('trajectories.trajectories') }}</h3>
          </v-card>
        </v-col>
      </v-row>
      <v-row no-gutters>
        <v-spacer>
          <div
            class="path"
            v-bind:style="{
              backgroundSize: `${this.rescaledWidth}px ${this.rescaledHeight}px`,
              backgroundRepeat: 'no-repeat',
              backgroundImage: `url('${this.tableImage}')`,
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
          </div>
        </v-spacer>
        <v-col sm="4">
          <Legend />
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Coordinate } from '@/types/coordinate';
import { mapState } from 'vuex';
import Legend from './Legend.vue';

@Component({
  components: {
    Legend: Legend,
  },
  computed: mapState([
    'tableImage',
    'plannedTrajectory',
    'currentPlannedTrajectory',
    'realTrajectory',
  ]),
})
export default class Trajectories extends Vue {
  private tableImage!: string;
  private plannedTrajectory!: Array<Coordinate>;
  private currentPlannedTrajectory!: Array<Coordinate>;
  private realTrajectory!: Array<Coordinate>;
  private readonly width = 1600;
  private readonly height = 904;
  private readonly ratioX = 0.45;
  private readonly ratioY = 0.45;

  private get rescaledWidth() {
    return this.width * this.ratioX;
  }

  private get rescaledHeight() {
    return this.height * this.ratioY;
  }

  private get plannedTrajectoryPoints() {
    return this.coordinatesToString(this.plannedTrajectory);
  }

  private get realTrajectoryPoints() {
    return this.coordinatesToString(this.realTrajectory);
  }

  private get startPointX() {
    return this.currentPlannedTrajectory[0]
      ? this.currentPlannedTrajectory[0].x * this.ratioX
      : 0;
  }

  private get startPointY() {
    return this.currentPlannedTrajectory[0]
      ? this.currentPlannedTrajectory[0].y * this.ratioY
      : 0;
  }

  private get destinationPointX() {
    return this.currentPlannedTrajectory[
      this.currentPlannedTrajectory.length - 1
    ]
      ? this.currentPlannedTrajectory[this.currentPlannedTrajectory.length - 1]
          .x * this.ratioX
      : 0;
  }

  private get destinationPointY() {
    return this.currentPlannedTrajectory[
      this.currentPlannedTrajectory.length - 1
    ]
      ? this.currentPlannedTrajectory[this.currentPlannedTrajectory.length - 1]
          .y * this.ratioY
      : 0;
  }

  private coordinatesToString(trajectory: Array<Coordinate>) {
    let points = '';
    trajectory.forEach(
      (coordinate) =>
        (points += `${coordinate.x * this.ratioX},${
          coordinate.y * this.ratioY
        } `)
    );
    return points;
  }
}
</script>

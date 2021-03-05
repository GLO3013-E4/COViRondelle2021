<template>
  <v-card
    class="d-flex justify-center mb-10"
    color="#ededed"
    :height="this.rescaleHeight + 70"
    :width="this.rescaleWidth"
  >
    <v-container>
      <v-row>
        <v-col sm="12">
          <v-card class="d-flex justify-center">
            <h3>Planned Trajectory</h3>
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
              :points="this.trajectoryPoints"
              style="fill: none; stroke: blue; stroke-width: 2"
            />
            <polyline
              id="real_path"
              :points="this.realTrajectoryPoints"
              style="fill: none; stroke: red; stroke-width: 2"
            />
            <circle
              class="name"
              :cx="startPoint.x * ratioX"
              :cy="startPoint.y * ratioY"
              r="2"
              stroke="red"
              stroke-width="2"
              fill="none"
            />
            }
          </svg>
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Coordinate } from '@/types/coordinate';
import { mapState } from 'vuex';

@Component({
  computed: {
    ...mapState(['tableImage', 'plannedTrajectory', 'realTrajectory']),
  },
})
export default class PlannedTrajectory extends Vue {
  private plannedTrajectory!: Array<Coordinate>;
  private realTrajectory!: Array<Coordinate>;
  private readonly tableImage!: string;
  private readonly width!: number;
  private readonly height!: number;
  private ratioX = 0.3;
  private ratioY = 0.3;
  private rescaleWidth!: number;
  private rescaleHeight!: number;
  // private startPoint!: Coordinate;

  public constructor() {
    super();

    this.width = 1600; // TODO : Get image width in computed
    this.rescaleWidth = this.width * this.ratioX;
    this.height = 904; // TODO : Get image height in computed
    this.rescaleHeight = this.height * this.ratioY;
  }

  private get trajectoryPoints() {
    let points = '';

    this.plannedTrajectory.forEach(
      (coordinate) =>
        (points += `${coordinate.x * this.ratioX},${
          coordinate.y * this.ratioY
        } `)
    );

    return points;
  }

  private get realTrajectoryPoints() {
    let points = '';

    this.realTrajectory.forEach(
      (coordinate) =>
        (points += `${coordinate.x * this.ratioX},${
          coordinate.y * this.ratioY
        } `)
    );

    return points;
  }
  private get startPoint() {
    return this.plannedTrajectory[0];
  }
}
</script>

<style>
#planned_path {
  z-index: 1;
}
</style>

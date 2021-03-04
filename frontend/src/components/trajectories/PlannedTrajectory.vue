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
        <v-col class="path"
          v-bind:style="{
            background: `url('${tableImage}')`,
            backgroundSize: `${this.rescaleWidth}px ${this.rescaleHeight}px`,
            backgroundRepeat: 'no-repeat',
          }">
          <svg :height="this.rescaleHeight" :width="this.rescaleWidth" id="svg">
            <polyline
              id="planned_path"
              :points = 'this.trajectoryPoint'
              style="fill: none; stroke: blue; stroke-width: 7"
            />
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
    ...mapState(['tableImage', 'plannedTrajectory']),
  },
})
export default class PlannedTrajectory extends Vue {
  private plannedTrajectory!: Array<Coordinate>;
  private readonly tableImage!: string;
  private readonly width!: number;
  private readonly height!: number;
  private ratioX = 0.3;
  private ratioY = 0.3;
  private rescaleWidth!: number;
  private rescaleHeight!: number;
  private trajectoryPoint !: string;
  public constructor() {
    super();

    this.width = 1600;
    this.rescaleWidth = this.width * this.ratioX;
    this.height = 904;
    this.rescaleHeight = this.height * this.ratioY;
    // this.rescaleCoordinates();
    this.coordinatesToString();
  }
  private coordinateToString(coordinate: Coordinate): string {
    return coordinate.x + ',' + coordinate.y + ' ';
  }
  private coordinatesToString() {
    // let res = '';
    // this.plannedTrajectory.forEach(
    //   (value) => (res += this.coordinateToString(value))
    // );
    // this.trajectoryPoint = res;
    this.trajectoryPoint = ' 80,120 200,180 200,230 60,230';
  }

  private applyRatio(coordinate: Coordinate): Coordinate {
    coordinate.x = coordinate.x * this.ratioX;
    coordinate.y = coordinate.y * this.ratioY;
    return coordinate;
  }

  private rescaleCoordinates() {
    this.plannedTrajectory = this.plannedTrajectory.map((value) => this.applyRatio(value));
  }
}
</script>

<style>
#planned_path {
  z-index: 1;
}
</style>

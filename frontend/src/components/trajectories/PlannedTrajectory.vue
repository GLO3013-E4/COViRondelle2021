<template>
  <v-card
    class="d-flex justify-center mb-10"
    color="#ededed"
    height="500"
    width="885"
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
            backgroundImage: 'url(' + this.image + ')',
            backgroundSize: rescaleWidth + 'px ' + rescaleHeight + 'px',
          }"
        >
          <svg :height="this.height" :width="this.width" id="svg">
            <polyline
              id="planned_path"
              :points="this.coordinatesToString"
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
    ...mapState(['image', 'coordinates']),
  },
})
export default class PlannedTrajectory extends Vue {
  private coordinates!: Array<Coordinate>;
  private readonly image!: string;
  private img;
  private readonly width!: number;
  private readonly height!: number;
  private ratioX = 0.1;
  private ratioY = 0.1;
  private rescaleWidth!: number;
  private rescaleHeight!: number;

  public constructor() {
    super();
    this.img = new Image();
    this.image = '../../../public/test.jpg';
    this.img.src = this.image;
    this.width = this.img.width;
    this.rescaleWidth = this.width * this.ratioX;
    this.height = this.img.height;
    this.rescaleHeight = this.height * this.ratioY;
    // this.rescaleCoordinates();
  }
  private static coordinateToString(coordinate: Coordinate): string {
    return coordinate.x + ',' + coordinate.y + ' ';
  }
  private coordinatesToString(): string {
    let res = '';
    this.coordinates.forEach(
      (value) => (res += PlannedTrajectory.coordinateToString(value))
    );
    return res;
  }

  private modifyValue(coordinate: Coordinate): Coordinate {
    coordinate.x = coordinate.x * this.ratioX;
    coordinate.y = coordinate.y * this.ratioY;
    return coordinate;
  }

  private rescaleCoordinates() {
    this.coordinates = this.coordinates.map((value) => this.modifyValue(value));
  }
  // data() {
  //   return {
  //     Point: '20,20 40,25 60,40 80,120 120,140 200,180',
  //   };
  // }
  mounted() {
    new PlannedTrajectory();
  }
}
</script>

<style>
/*.path {*/
/*  background: url('../../../public/test.jpg');*/
/*}*/
#planned_path {
  z-index: 1;
}
</style>

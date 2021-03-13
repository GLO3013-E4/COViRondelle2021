<template>
  <v-card
    class="d-flex justify-center mb-10"
    color="#ededed"
    :height="this.rescaleHeight + 150"
    :width="this.rescaleWidth"
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
              :points="this.trajectoryPoints"
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
            <svg height="100" width="200" id="legend">
              <text x="0" y="15" fill="blue">
                {{ `${$t('trajectories.plannedTrajectory')} :` }}
              </text>
              <line
                x1="150"
                y1="10"
                x2="200"
                y2="10"
                style="stroke: blue; stroke-width: 2"
              />
              <text x="0" y="35" fill="red">
                {{ `${$t('trajectories.realTrajectory')} :` }}
              </text>
              <line
                x1="150"
                y1="30"
                x2="200"
                y2="30"
                style="stroke: red; stroke-width: 2"
              />
              <text x="0" y="55" fill="red">
                {{ `${$t('trajectories.startingPoint')} :` }}
              </text>
              <circle
                cx="150"
                cy="50"
                r="2"
                stroke="red"
                stroke-width="2"
                fill="none"
              />
              <text x="0" y="75" fill="green">
                {{ `${$t('trajectories.destinationPoint')} :` }}
              </text>
              <circle
                cx="150"
                cy="70"
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

@Component({
  computed: {
    ...mapState(['tableImage', 'plannedTrajectory', 'realTrajectory']),
  },
})
export default class Trajectories extends Vue {
  private plannedTrajectory!: Array<Coordinate> | undefined;
  private realTrajectory!: Array<Coordinate> | undefined;
  private readonly tableImage!: string;
  private readonly width!: number;
  private readonly height!: number;
  private ratioX = 0.3;
  private ratioY = 0.3;
  private rescaleWidth!: number;
  private rescaleHeight!: number;

  public constructor() {
    super();

    this.width = 1600; // TODO : Get image width in computed
    this.rescaleWidth = this.width * this.ratioX;
    this.height = 904; // TODO : Get image height in computed
    this.rescaleHeight = this.height * this.ratioY;
  }
  private coordinatesToString(isRealTrajectory: boolean) {
    let points = '';
    const trajectory = isRealTrajectory
      ? this.realTrajectory
      : this.plannedTrajectory;
    if (trajectory !== undefined) {
      trajectory.forEach(
        (coordinate) =>
          (points += `${coordinate.x * this.ratioX},${
            coordinate.y * this.ratioY
          } `)
      );
      this.newTrajectorySize += trajectory.length;
    }
    return this.updateTrajectories(points, isRealTrajectory);
  }
  private get trajectoryPoints() {
    return this.coordinatesToString(false);
  }

  private get realTrajectoryPoints() {
    return this.coordinatesToString(true);
  }
  private get startPointX() {
    return this.plannedTrajectory[0] !== undefined
      ? this.plannedTrajectory[0].x * this.ratioX
      : 0;
  }
  private get startPointY() {
    return this.plannedTrajectory[0] !== undefined
      ? this.plannedTrajectory[0].y * this.ratioY
      : 0;
  }
  private get destinationPointX() {
    return this.plannedTrajectory[this.plannedTrajectory.length - 1] !==
      undefined
      ? this.plannedTrajectory[this.plannedTrajectory.length - 1].x *
          this.ratioX
      : 0;
  }
  private get destinationPointY() {
    return this.plannedTrajectory[this.plannedTrajectory.length - 1] !==
      undefined
      ? this.plannedTrajectory[this.plannedTrajectory.length - 1].y *
          this.ratioY
      : 0;
  }
  private updateTrajectories(points: string, isRealTrajectory: boolean) {
    const trajectoryType = isRealTrajectory
      ? 'realTrajectories'
      : 'plannedTrajectories';
    let actualPoints = localStorage.getItem(trajectoryType);
    if (!actualPoints) {
      localStorage.setItem(trajectoryType, points);
      return points;
    } else {
      actualPoints += points;
      localStorage.setItem(trajectoryType, actualPoints);
      return actualPoints;
    }
  }
  mounted() {
    // Clear the browser cache data in localStorage when closing the browser window
    window.onbeforeunload = () => {
      localStorage.clear();
    };
  }
}
</script>

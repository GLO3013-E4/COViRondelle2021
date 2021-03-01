<template>
  <v-card class="d-flex justify-center mb-10" color="#ededed" height="500" width="885">
    <v-container>
      <v-row>
        <v-col sm="12">
          <v-card class="d-flex justify-center">
            <h3>Planned Trajectory</h3>
          </v-card>
        </v-col>
      </v-row>
      <v-row no-gutters>
        <v-col class="path">
          <svg height="500" width="885" id="svg">
            <polyline
              id="planned_path"
              :points="Point"
              style="fill: none; stroke: blue; stroke-width: 7"
            />
          </svg>
        </v-col>
      </v-row>
    </v-container>
    <!--     <div class="traj" v-for="point in Point" :key="point">-->
    <!--       <div class="box">Just testing this {{point.x}}}</div>-->
    <!--     </div>-->
    <!--    <svg height="200" width="500">-->
    <!--      <polyline points="0,0 40,25 60,40 80,120 120,140 200,180" style="fill:none;stroke:black;stroke-width:3" />-->
    <!--    </svg>-->
    <!--    <canvas id="canvas" width="250" height="250" style="border: 1px solid black;"></canvas>-->
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator'
@Component({
  // computed: {
  //   ...mapState('drawLines'),
  // },
})
export default class PlannedTrajectory extends Vue {
  data() {
    return {
      Point:
        '20,20 40,25 60,40 80,120 120,140 200,180',
    };
  }
}
</script>

<style>
.path {
  background: url('../../../public/test.jpg');
}
#planned_path{
  z-index: 1;
}
</style>

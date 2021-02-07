<template>
  <v-card width="90%" height="700">
    <v-row>
      <v-col>
        <CyclesInformations />
      </v-col>
      <v-col>
        <StationInformations />
      </v-col>
    </v-row>
    <v-row>
      <v-col>
        <Consumptions />
      </v-col>
      <v-col>
        <div class="d-flex flex-column">
          <PlannedTrajectory />
          <RealTrajectory />
        </div>
      </v-col>
    </v-row>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from "vue-property-decorator";
import Consumptions from "../components/Consumptions/Consumptions.vue";
import CyclesInformations from "../components/CyclesInformations/CyclesInformations.vue";
import PlannedTrajectory from "../components/PlannedTrajectory/PlannedTrajectory.vue";
import RealTrajectory from "../components/RealTrajectory/RealTrajectory.vue";
import StationInformations from "../components/StationInformations/StationInformations.vue";

@Component({
  components: {
    Consumptions,
    CyclesInformations,
    StationInformations,
    PlannedTrajectory,
    RealTrajectory,
  },
})
export default class RobotInformations extends Vue {}
</script>

<style></style>

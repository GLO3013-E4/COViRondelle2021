<template>
  <v-card width="100%" height="700">
    <v-row>
      <v-col>
        <!-- TODO : Layout should have components injected. The main view should add these components -->
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
import Consumptions from "../components/consumptions/ConsumptionList.vue";
import CyclesInformations from "../components/cycles/CyclesInformations.vue";
import PlannedTrajectory from "../components/trajectories/RealTrajectory.vue";
import RealTrajectory from "../components/trajectories/RealTrajectory.vue";
import StationInformations from "../components/station/StationInformations.vue";

@Component({
  components: {
    Consumptions,
    CyclesInformations,
    StationInformations,
    PlannedTrajectory,
    RealTrajectory
  }
})
export default class RobotInformation extends Vue {}
</script>

<style></style>

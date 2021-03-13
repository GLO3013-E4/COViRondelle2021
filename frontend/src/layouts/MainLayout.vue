<template>
  <v-card width="100%" height="100%">
    <v-row>
      <v-col>
        <!-- TODO : Layout should have components injected. The main view should add these components -->
        <cycle-information />
      </v-col>
      <v-col>
        <station-information />
        <consumption-information />
      </v-col>
    </v-row>
    <v-row>
      <v-col> </v-col>
      <v-col>
        <div class="d-flex flex-column">
          <planned-trajectory />
          <real-trajectory />
        </div>
      </v-col>
    </v-row>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import ConsumptionInformation from '../components/consumptions/ConsumptionInformation.vue';
import CycleInformation from '../components/cycles/CycleInformation.vue';
import PlannedTrajectory from '../components/trajectories/PlannedTrajectory.vue';
import RealTrajectory from '../components/trajectories/RealTrajectory.vue';
import StationInformation from '../components/station/StationInformation.vue';

@Component({
  components: {
    ConsumptionInformation,
    CycleInformation,
    StationInformation,
    PlannedTrajectory,
    RealTrajectory,
  },
})
export default class MainLayout extends Vue {}
</script>

<style></style>

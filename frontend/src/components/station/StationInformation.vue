<template>
  <v-card color="#ededed" height="250">
    <v-container>
      <v-row>
        <v-col sm="12">
          <v-card class="d-flex justify-center">
            <h3>{{ $t('station.stationInformation') }}</h3>
          </v-card>
        </v-col>
      </v-row>
      <v-row no-gutters>
        <v-col sm="6">
          <Resistance />
        </v-col>
        <v-col sm="6">
          <ControlPanel />
        </v-col>
        <v-col sm="6">
          <PuckInGrip />
        </v-col>
        <v-col sm="6">
          <PuckDeposit />
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import Resistance from '../station/Resistance.vue';
import ControlPanel from '../station/ControlPanel.vue';
import PuckInGrip from '../station/PuckInGrip.vue';
import PuckDeposit from '../station/PuckDeposit.vue';

@Component({
  components: {
    Resistance: Resistance,
    ControlPanel: ControlPanel,
    PuckInGrip: PuckInGrip,
    PuckDeposit: PuckDeposit,
  },
})
export default class StationInformation extends Vue {}
</script>

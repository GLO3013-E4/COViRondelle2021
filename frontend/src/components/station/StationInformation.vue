<template>
  <v-card color="#ededed" height="230">
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
          <FirstCorner />
        </v-col>
        <v-col sm="6">
          <GripState />
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
import FirstCorner from './FirstCorner.vue';
import GripState from './GripState.vue';
import PuckDeposit from '../station/PuckDeposit.vue';

@Component({
  components: {
    Resistance: Resistance,
    FirstCorner: FirstCorner,
    GripState: GripState,
    PuckDeposit: PuckDeposit,
  },
})
export default class StationInformation extends Vue {}
</script>

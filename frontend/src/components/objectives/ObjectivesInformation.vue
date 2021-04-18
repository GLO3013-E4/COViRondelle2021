<template>
  <v-card color="base">
    <v-container>
      <v-row>
        <v-col sm="12">
          <v-card class="d-flex justify-center">
            <h3>{{ $t('objectives.objectivesInformation') }}</h3>
          </v-card>
        </v-col>
      </v-row>
      <v-row>
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
import Resistance from './/Resistance.vue';
import FirstCorner from './FirstCorner.vue';
import GripState from './GripState.vue';
import PuckDeposit from './/PuckDeposit.vue';

@Component({
  components: {
    Resistance: Resistance,
    FirstCorner: FirstCorner,
    GripState: GripState,
    PuckDeposit: PuckDeposit,
  },
})
export default class ObjectivesInformation extends Vue {}
</script>

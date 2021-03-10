<template>
  <v-card class="d-flex justify-center" color="#ededed" height="245">
    <v-container>
      <v-row>
        <v-col sm="12">
          <v-card class="d-flex justify-center">
            <h3>{{ $t('consumptions.consumptionInformation') }}</h3>
          </v-card>
        </v-col>
      </v-row>
      <v-row no-gutters>
        <v-col sm="6">
          <BatteryCharge />
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import BatteryCharge from './BatteryCharge.vue';

@Component({
  components: { BatteryCharge: BatteryCharge },
})
export default class ConsumptionInformation extends Vue {}
</script>

<style></style>

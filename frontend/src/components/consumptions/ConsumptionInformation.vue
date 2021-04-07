<template>
  <v-card class="d-flex justify-center" color="base" height="245">
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
        <v-col sm="6">
          <RobotConsumptionInfo />
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import BatteryCharge from './BatteryCharge.vue';
import RobotConsumptionInfo from './RobotConsumptionInfo.vue';

@Component({
  components: {
    BatteryCharge: BatteryCharge,
    RobotConsumptionInfo: RobotConsumptionInfo,
  },
})
export default class ConsumptionInformation extends Vue {}
</script>

<style></style>

<template>
  <v-card color="base">
    <v-container>
      <v-row>
        <v-col sm="12">
          <v-card class="d-flex justify-center">
            <h3>{{ $t('cycles.cycleInformation') }}</h3>
          </v-card>
        </v-col>
      </v-row>
      <v-row>
        <v-col sm="8" class="stepList">
          <StepList />
        </v-col>
        <v-spacer></v-spacer>
        <v-col sm="4">
          <v-container>
            <v-row>
              <v-col class="cycleNumberCol">
                <CycleNumber />
              </v-col>
            </v-row>
            <v-row>
              <v-col>
                <Mode />
              </v-col>
            </v-row>
            <v-row>
              <v-col>
                <Chronometer />
              </v-col>
            </v-row>
          </v-container>
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import CycleNumber from "@/components/cycles/CycleNumber.vue";
import StepList from '../cycles/StepList.vue';
import Mode from '../cycles/Mode.vue';
import Chronometer from '../cycles/Chronometer.vue';

@Component({
  components: {
    CycleNumber,
    StepList,
    Mode,
    Chronometer,
  },
})
export default class CycleInformation extends Vue {}
</script>

<style>
.stepList {
  padding-right: 1em !important;
}
.cycleNumberCol {
  padding-top: 0 !important;
}
</style>

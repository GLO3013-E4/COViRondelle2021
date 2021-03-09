<template>
  <v-card color="#ededed" height="475">
    <v-container>
      <v-row>
        <v-col sm="12">
          <v-card class="d-flex justify-center">
            <h3>{{ $t('cycles.cycleInformation') }}</h3>
          </v-card>
        </v-col>
      </v-row>
      <v-row no-gutters>
        <v-col sm="8">
          <StepList />
        </v-col>
        <v-col sm="4">
          <Mode />
          <Chronometer />
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import StepList from '../cycles/StepList.vue';
import Mode from '../cycles/Mode.vue';
import Chronometer from '../cycles/Chronometer.vue';

@Component({
  components: {
    StepList: StepList,
    Mode: Mode,
    Chronometer: Chronometer,
  },
})
export default class CycleInformation extends Vue {}
</script>

<style></style>

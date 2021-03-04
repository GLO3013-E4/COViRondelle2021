<template>
  <div>
    <v-btn @click="start" width="100%" elevation="3"> start </v-btn>
  </div>
</template>

<script lang="ts">
import { Step } from '@/types/step';
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';

@Component({
  computed: {
    ...mapState(['cycleReady', 'currentStep']),
  },
})
export default class StartButton extends Vue {
  public cycleReady!: boolean;
  public currentStep!: Step;

  public start() {
    this.$emit('start');
  }
}
</script>

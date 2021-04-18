<template>
  <div class="padding-list">
    <v-stepper v-model="currentStepNumber" vertical>
      <v-stepper-step
        ref="step"
        v-for="(step, i) in steps"
        :key="i"
        :complete="currentStepNumber > i + 1"
        :step="i + 1"
        :color="currentStepNumber === i + 1 ? 'blue' : 'green'"
      >
        {{ $t(`cycles.steps.${step}`) }}
      </v-stepper-step>
    </v-stepper>
  </div>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';
import { Step } from '@/types/step';

@Component({
  components: {},
  computed: {
    ...mapState(['currentStep']),
  },
})
export default class StepList extends Vue {
  public currentStep!: Step;

  get currentStepNumber(): number {
    return this.currentStep.valueOf() + 1;
  }

  get steps(): Array<string> {
    const result = [];
    for (const step in Step) {
      const isValueProperty = parseInt(step.valueOf(), 10) >= 0;
      if (isValueProperty) {
        result.push(Step[step]);
      }
    }
    return result;
  }
}
</script>

<style scoped>
.v-stepper--vertical .v-stepper__step {
  padding: 0.05em 0.3em 0.15em;
}
.v-stepper--vertical {
  padding: 0.2em;
}
</style>

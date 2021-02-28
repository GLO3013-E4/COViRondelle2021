<template>
  <div>
    <v-stepper v-model="e6" vertical>
      <v-stepper-step
        v-for="(step, i) in steps"
        :key="i"
        :complete="e6 > 3"
        :step="i + 1"
      >
        {{ step }}
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
  public e6 = 1; //TODO: to change

  get steps(): Array<any> {
    const result = [];
    for (const step in Step) {
      const isValueProperty = parseInt(step, 10) >= 0;
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
  padding: 1px 0px 0px;
}
.v-stepper--vertical{
    padding: 0px;
}
</style>

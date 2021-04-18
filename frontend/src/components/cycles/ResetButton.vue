<template>
  <div>
    <v-btn
      :disabled="this.isDisabled"
      color="secondary"
      @click="reset"
      width="100%"
      elevation="6"
    >
      {{ $t('cycles.reset') }}
    </v-btn>
  </div>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';
import { Step } from '@/types/step';

@Component({
  computed: {
    ...mapState(['currentStep']),
  },
})
export default class ResetButton extends Vue {
  public currentStep!: Step;

  public reset() {
    this.$emit('reset');
  }

  get isDisabled() {
    return this.currentStep !== Step.CycleEndedAndRedLedOn;
  }
}
</script>

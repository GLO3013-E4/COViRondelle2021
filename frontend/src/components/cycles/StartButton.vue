<template>
  <div>
    <v-btn @click="start" width="100%"> start </v-btn>
  </div>
</template>

<script lang="ts">
//TODO: COMPLETE COMPONENT
import { Step } from '@/types/step';
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';

@Component({
   computed: {
    ...mapState(['cycleReady', 'currentStep']),
  },
})
export default class StartButton extends Vue {
  public cycleReady!:boolean;
  public currentStep!:Step;

  public start() {
    this.$emit('start');
  }

  get color(){
    if(this.cycleReady || this.currentStep == Step.CycleEndedAndRedLedOn){
      return 'green'
    }
    else{
      return "white"
    }
  }
}
</script>

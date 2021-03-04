<template>
    <div class="d-flex justify-center" >
    <v-chip :color="this.color" class="white--text d-flex justify-center" ref="mode"><h3>{{this.actualMode}}</h3></v-chip>
    </div>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';

@Component({
  computed: {
    ...mapState(['cycleReady', 'cycleStarted']),
  },
})
export default class Mode extends Vue {
  public cycleReady!: boolean;
  public cycleStarted!: boolean;

  get actualMode(){
    if(!this.cycleReady){
      return "Booting"
    }
    else if(this.cycleReady && this.cycleStarted){
      return "Started"
    }
    else if(this.cycleReady){
      return "Waiting"
    }
    return "no information"
  }

  get color(){
  if(!this.cycleReady){
      return "red"
    }
    if(this.cycleReady && this.cycleStarted){
      return "green"
    }
    else if(this.cycleReady){
      return "amber"
    }
    return "white"
  }
}
</script>

<style>
.v-chip{
  width: 100%
}
</style>
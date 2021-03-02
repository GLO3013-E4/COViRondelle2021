<template>
  <div>
    <v-btn @click="start"> start </v-btn>
  </div>
</template>

<script lang="ts">
//TODO: COMPLETE COMPONENT
import { Component, Vue } from 'vue-property-decorator';

@Component({})
export default class StartButton extends Vue {
  public start() {
    this.$emit('start');
  }
}
</script>

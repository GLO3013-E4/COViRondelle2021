<template>
  <div>
    <v-btn color="accent" @click="start" width="100%" elevation="6">{{
      $t('cycles.start')
    }}</v-btn>
  </div>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';

@Component({})
export default class StartButton extends Vue {
  public start() {
    this.$emit('start');
  }
}
</script>

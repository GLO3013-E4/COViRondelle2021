<template>
  <div>
    <!-- TODO: Fix color -->
    <v-btn color="secondary" @click="reset" width="100%" elevation="6">{{
      $t('cycles.reset')
    }}</v-btn>
  </div>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';

// TODO : Disable if step is not CycleEnded
@Component({})
export default class ResetButton extends Vue {
  public reset() {
    this.$emit('reset');
  }
}
</script>

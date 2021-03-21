<template>
  <v-card class="lighten2">
    <v-card-title sm="6" class="lighten1 d-flex justify-center">
      <h5 class="white--text">{{ $t('station.gripState') }}</h5>
    </v-card-title>
    <div
      ref="gripState"
      class="d-flex justify-center font-weight-bold"
      id="divChip"
    >
      <v-chip :color="puckInGrip ? 'green' : 'red'">
        <h3 class="white--text">
          {{ puckInGrip ? $t('station.puckInGrip') : $t('station.noPuck') }}
        </h3></v-chip
      >
    </div>
  </v-card>
</template>
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';

@Component({
  computed: {
    ...mapState(['puckInGrip']),
  },
})
export default class GripState extends Vue {
  public puckInGrip!: boolean;
}
</script>

<style scoped>
.v-card__title {
  padding: 0;
}
#divChip {
  padding: 0.3em;
}
</style>

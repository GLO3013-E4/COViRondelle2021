<template>
  <v-card class="grey lighten-3">
    <v-card-title sm="6" class="grey darken-1 d-flex justify-center">
      <h5 class="white--text">Grip state</h5>
    </v-card-title>
    <div ref="gripState" class="d-flex justify-center font-weight-bold" id="divChip">
      <v-chip :color="puckInGrip ? 'green' : 'gray'">
        <h3 class="white--text">
          {{ puckInGrip ? 'puck in grip' : 'no puck' }}
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
export default class PuckInGrip extends Vue {
  public puckInGrip!: boolean;
}
</script>

<style scoped>
.v-card__title {
  padding: 0px;
}
#divChip{
  padding:0.3em;
}


</style>
